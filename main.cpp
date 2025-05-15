#include <libhackrf/hackrf.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <math.h>
#include <fstream>
#include <future>
#include "audio_utils.h"
#include "buffer.h"
#include "Mode_switcher.hpp"

//для rx_callback
#define FREQ  405000000
#define BUF_LENGTH 262144
const unsigned int sample_rate_hz = 2'400'000;
extern const int hackrf_i_q_size = BUF_LENGTH / 2;
signed char hackrf_i[hackrf_i_q_size];			//������ �������� ��������� [131 072]
signed char hackrf_q[hackrf_i_q_size];
int DC_shift = 16;	
int accum_i = 0, accum_q = 0;
double accum_demod = 0;
const int len_fir_imp = 23;
double tail_i_i[len_fir_imp] = { 0 } , tail_q[len_fir_imp] = { 0 }, tail_demod1[len_fir_imp] = { 0 };
int samples_after_dec_demod = 1, samples_after_dec_i = 1, samples_after_dec_q = 1;
//tail_demod[23] = { 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150 };
int length_tail_demod = len_fir_imp,						length_tail_i = len_fir_imp,					length_tail_q = len_fir_imp;
int ind_demod = len_fir_imp,	ind_i = len_fir_imp,	ind_q = len_fir_imp; //+(len_fir_imp-1)/2 ����� ��� ���� ����� ������ �������� ����� ������� ������� ��� ��� � ����� (len_fir_imp-1)/2
const int hackrf_i_q_shift_size = BUF_LENGTH / 2;
double hackrf_i_shift[hackrf_i_q_shift_size];	//������ ��������� ����� ��������	[131 072]
double hackrf_q_shift[hackrf_i_q_shift_size];

int shift_freq_hz=-192000;

const int demod_samples_size = 2000;
const int demod_dec_size = 2000;
double demod_samples[demod_samples_size];		//����� ���������������� �������� �������� [1313]
double demod_dec[demod_dec_size];
int16_t demod_dec_int[demod_dec_size];
double fir_imp_koef[len_fir_imp] = { 1 };				//������ ������������� ��
double fir_imp_koef1[len_fir_imp] = { 1 };
const int hackrf_i_q_dec1_size = 20000;
double hackrf_i_dec1[hackrf_i_q_dec1_size];		//����� ������ ���������	[13 1 10]
double hackrf_q_dec1[hackrf_i_q_dec1_size];
const int length_merge_i = 200000;    //������ �������������� ������� ��� ������� � ���������
const int length_merge_q = 200000;
const int length_merge_demod = 200000;
double merge_arr_i[length_merge_i];
double merge_arr_q[length_merge_q];
double merge_arr_demod[length_merge_demod];
typedef enum {
	TRANSCEIVER_MODE_OFF = 0,
	TRANSCEIVER_MODE_RX = 1,
	TRANSCEIVER_MODE_TX = 2,
} transceiver_mode_t;
static transceiver_mode_t transceiver_mode = TRANSCEIVER_MODE_TX;
//
//FILE* iq;

Mode_switcher mode_switcher;

std::atomic<bool> stop_flag(false);

const size_t FRAME_SIZE = 1311; // 4096 семплов на фрейм
SharedAudioBuffer recorder_buffer(FRAME_SIZE * 2 * 100); // 10 фреймов по 4096 семплов (S16)
SharedAudioBuffer player_buffer(FRAME_SIZE * 2 * 100); // 10 фреймов по 4096 семплов (S16)


std::thread recorder_thread;
std::thread player_thread;

hackrf_device* device = NULL;

//для tx_callback
double integrator = 0;
short compressor[1311];	

void record_thread_func() {
    snd_pcm_t* capture = init_audio("default", SND_PCM_STREAM_CAPTURE);
    std::vector<char> buffer(FRAME_SIZE * 2); // S16 -> 2 байта на семпл

    while (!stop_flag) {
        int err = snd_pcm_readi(capture, buffer.data(), FRAME_SIZE);
        
        if (stop_flag) {
            // Нет доступных данных — немного спим и продолжаем
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        if (err == -EPIPE) {
            snd_pcm_recover(capture, err, 1);
        } else if (err < 0) {
            std::cerr << "[Record] ALSA read error: " << snd_strerror(err) << std::endl;
            break;
        }

        if (err > 0) {
            recorder_buffer.write(buffer.data(), buffer.size());
        }
    }
    deinit_audio(capture);
    std::cout << "[Record] Запись завершена.\n";
}
void force_stop_threads() {
    std::cout << "[FORCE STOP] Принудительное завершение потоков...\n";

    stop_flag = true;

    // Останавливаем буфер
    player_buffer.stop();
	recorder_buffer.stop();

    // Ждём завершения потоков с таймаутом
    if (recorder_thread.joinable()) {
        auto recorder_future = std::async(std::launch::async, [&] {
            recorder_thread.join();
        });

        if (recorder_future.wait_for(std::chrono::seconds(2)) == std::future_status::timeout) {
            std::cerr << "[ERROR] Таймаут при ожидании завершения recorder_thread\n";
            recorder_thread.detach(); // Отсоединяем, чтобы не вызвать terminate()
        }
    }

    if (player_thread.joinable()) {
        auto player_future = std::async(std::launch::async, [&] {
            player_thread.join();
        });

        if (player_future.wait_for(std::chrono::seconds(2)) == std::future_status::timeout) {
            std::cerr << "[ERROR] Таймаут при ожидании завершения player_thread\n";
            player_thread.detach(); // Отсоединяем
        }
    }

    std::exit(EXIT_SUCCESS);
}
int fm_tx_callback(hackrf_transfer* transfer) {
	static const int amp = 115;
	static const int resampling_factor = 100; //50 ��� 48000 samplerate, 100 ��� 24000 samplerate
	double deviation = 1.0 / 50;
	//static int8_t current_i = amp;
	//static int8_t current_q = 0;
	static int sample_repeats;
	sample_repeats = resampling_factor;
	int counter = 0;
	double max=0;

	size_t samples_to_fill;
	unsigned int i = 0;
	short snd_in = 0;  //����
	samples_to_fill = transfer->valid_length / 2;

	int file_buff_ind = 0;
	int iter = samples_to_fill;

    recorder_buffer.read(compressor,1311*2);
	for (int i = 0; i < 1311; i++) {
		//std::cout<<compressor[i]<<std::endl;
		//std::this_thread::sleep_for(std::chrono::microseconds(10));
		if (abs(compressor[i]) > max) {
			max = abs(compressor[i]);
		}

	}
	if (max < 100) {
		max = 32767;
	}
	//std::cout << "max=" << max * 1.0 << std::endl;

	while (samples_to_fill)			//���� ���� ����� � ������ ���� ��� ����������
	{
		if (!recorder_buffer.empty())	//���� ���� �������� � �������
		{
			snd_in = compressor[counter];	//����� �� ������ ������� � �������� ������
			//snd_in = 0;
			//mic_gainer(snd_in);
		}
		else
		{
			snd_in = 0;		//���� ��� ����� ����
		}
		while (sample_repeats--)	//��������� ������ ����� 100 ���
		{	
			//��������� ���������� �������� �������
			integrator += (double)snd_in * deviation / max;
			transfer->buffer[i++] = (int8_t)(amp * cos(integrator));
			//quadrat[i-1]=transfer->buffer[i-1];
            //std::cout<<(int)transfer->buffer[i-1]<<std::endl;
			transfer->buffer[i++] = (int8_t)(amp * sin(integrator));
			//quadrat[i-1]=transfer->buffer[i-1];
            //std::cout<<(int)transfer->buffer[i-1]<<std::endl;
			samples_to_fill -= 1;
			if (samples_to_fill == 0) //���� ����� � ������ ���� ��������� - �����
			{
				return 0;
			}

		}

		//��������� ���������� �������� �������
		//integrator += (snd_in/2000.0) * deviation; //��������������
		//current_i = amp * cos(integrator);
		//current_q = amp * sin(integrator);
		//��������� ����� �������� ������������� ������������
		sample_repeats = resampling_factor;
		counter++;
		//std::cout << "counter=" << counter << std::endl;
	}
	//fwrite(quadrat,sizeof(uint8_t),262144,iq);
	return 0;
}
void playback_thread_func() {
    snd_pcm_t* playback = init_audio("default", SND_PCM_STREAM_PLAYBACK);
    std::vector<char> buffer(FRAME_SIZE * 2);

    while (!stop_flag) {
        player_buffer.read(buffer.data(), buffer.size());
        int err = snd_pcm_writei(playback, buffer.data(), FRAME_SIZE);
        if (err == -EPIPE) {
            snd_pcm_recover(playback, err, 1);
        } else if (err < 0) {
            std::cerr << "[Playback] ALSA write error: " << snd_strerror(err) << std::endl;
            break;
        }
    }
    deinit_audio(playback);
    std::cout << "[Playback] Воспроизведение завершено.\n";
}

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\n[Сигнал] Ctrl+C нажат. Выход из цикла..." << std::endl;
        stop_flag=true;
    }
}
void decompose_iq(signed char* hackrf_buff_iq, signed char* hackrf_i, signed char* hackrf_q,int length) {
	for (int i = 0, c = 0; i < length; i += 2) {
		hackrf_i[c] = hackrf_buff_iq[i];                  ///
		hackrf_q[c] = hackrf_buff_iq[i + 1];                /////
		++c;
	}
}
//������� �� ������� ������� � �������������
void shift_iq(signed char* hackrf_i, signed char* hackrf_q, double* hackrf_i_shift, double* hackrf_q_shift, int f_shift) {
	for (int i = 0; i < BUF_LENGTH / 2; i++) {
		//double testCos = cos(2 * M_PI * f_shift * i * 1. / sample_rate_hz);
		hackrf_i_shift[i] = hackrf_i[i] * cos(2 * M_PI * f_shift * i * 1. / sample_rate_hz) - hackrf_q[i] * sin(2 * M_PI * f_shift * i * 1. / sample_rate_hz);
		hackrf_q_shift[i] = hackrf_i[i] * sin(2 * M_PI * f_shift * i * 1. / sample_rate_hz) + hackrf_q[i] * cos(2 * M_PI * f_shift * i * 1. / sample_rate_hz);
	}
}
//�� �����������
void fm_demod(double* in_i, double* in_q, double* demod_snd, const int count) {
	double k = 0.075; //����� ��������, �� ���� �������� �� ����� �������� ����� ��� ��� �����
	for (int i = 0; i < count-1; i++) {
		//demod_snd[i] = k*(in_i[i] * (in_q[i] - in_q[i - 1]) - in_q[i] * (in_i[i] - in_i[i - 1])); // pow(in_i[i], 2) + pow(in_q[i], 2);
		demod_snd[i] = k * (in_i[i+1] * (in_q[i+1] - in_q[i]) - in_q[i+1] * (in_i[i+1] - in_i[i]));
	}
}
void load_fir_imp(char* filepath, const int length, double* fir_imp_array) {
	std::ifstream fir_imp_file(filepath); // �������� ���� ��� ������
	if (!fir_imp_file.is_open()) {
		printf("load_fir_imp ERROR open file!\n");
		exit(EXIT_FAILURE);
	}
	for (int i = 0; i < length + 1; i++) {
		fir_imp_file >> fir_imp_array[i];
		//std::cout << fir_imp_array[i]<<std::endl;
	}
	fir_imp_file.close();
}
void DC_remove_byte(signed char* buffer, int dc_shift, int count,int* accum)
{
	int summ = *accum;
	for (int i = 0; i < count; i++) 
	{
		buffer[i] = buffer[i] - summ / pow(2, dc_shift);
		summ += buffer[i];
	}
	*accum = summ;
	
}
void DC_remove_double(double* buffer, int dc_shift, int count,double* accum)
{
	double summ = *accum;
	for (int i = 0; i < count; i++)
	{
		buffer[i] = buffer[i] - summ / pow(2, dc_shift);
		summ += buffer[i];
	}
	*accum = summ;
}
void conv_dec2(double* tail, int* length_tail, double* arr, const int length_arr, int* ind, int dec_rate, double *fir, int length_fir, int* length_out, double* out,double* merge_arr)
{	
	//std::cout<<"conv dec start"<<std::endl;
	//memset(out, 0, (*length_out) * sizeof(double));
	for (int i = 0; i < *length_out; i++)
	{
		out[i] = 0;
	}
	for (int i = 0; i < *length_tail; i++)
	{
		merge_arr[i] = tail[i];
	}
	for (int j = 0; j < length_arr; j++)
	{
		merge_arr[*length_tail + j] = arr[j];
	}
	//std::cout<<"merge arr written"<<std::endl;
	int length_merge = *length_tail + length_arr;
	//std::cout << length_merge << " length_merge" << std::endl;
	int last_ind = 0;
	int counter = 0;
	for (int h = *ind; h < length_merge; h += dec_rate)
	{
		for (int n = 0; n < length_fir; n++)
		{
			out[(h - *ind) / dec_rate] += merge_arr[h - n] * fir[n];
		}
		last_ind = h - *length_tail;
		counter++;
	}
	//std::cout<<"out buffer written"<<std::endl;
	//std::cout << last_ind << "= last_ind" << std::endl;;
	*length_out = counter;
	*length_tail = length_fir - dec_rate + length_arr - last_ind;
	int iter = length_arr - *length_tail - 1;
	for (int i = iter; i < length_arr; i++)
	{
		tail[i - iter] = arr[i];
	}
	//*ind = length_fir;
	//std::cout<<"tail buff written"<<std::endl;
}
double avg_level(signed char buffer_i[], signed char buffer_q[]) {
	double level = 0;
	for (int i = 0; i < 131072; i++) {
		level += pow(buffer_i[i],2)+pow(buffer_q[i], 2);
	}
	level = level / 131072.0;
	return level;
}
int rx_callback(hackrf_transfer* transfer) {
	//unsigned int bytes_written = fwrite(transfer->buffer, 1, transfer->buffer_length, file_rx_iq);
	
	//���������� ������� ����� hackrf �� ��� ������ I � Q
	//std::cout<<"start rx_callback"<<std::endl;
	decompose_iq((int8_t*)transfer->buffer, hackrf_i, hackrf_q,BUF_LENGTH);
	//std::cout<<"decompose done"<<std::endl;
	//std::cout << "accum_i=                         " << accum_i << std::endl;
	DC_remove_byte(hackrf_i, DC_shift, hackrf_i_q_size,&accum_i);
	//std::cout << "accum_q=                         " << accum_q << std::endl;
	DC_remove_byte(hackrf_q, DC_shift, hackrf_i_q_size,&accum_q);
	//std::cout<<"dc_remove done"<<std::endl;
	//����� ������������ �������
	shift_iq(hackrf_i, hackrf_q, hackrf_i_shift, hackrf_q_shift, shift_freq_hz);
	//std::cout<<"shift iq done"<<std::endl;

	//������� � ����������

	/*memset(hackrf_q_dec1, 0, hackrf_i_q_dec1_size * sizeof(double));
	memset(hackrf_i_dec1, 0, hackrf_i_q_dec1_size * sizeof(double));
	memset(hackrf_q_dec2, 0, hackrf_i_q_dec2_size * sizeof(double));
	memset(hackrf_i_dec2, 0, hackrf_i_q_dec2_size * sizeof(double));*/

	//������ ������� ���������� � ��������� � 10 ���
	//std::cout << "i1" << std::endl;
	//conv_dec1(hackrf_i_shift, hackrf_i_q_shift_size, fir_imp_koef, len_fir_imp, 10, &ind_i, hackrf_i_dec1, tail_i, &length_tail_i);

	//std::cout<< samples_after_dec_i << " &samples_after_dec_i" << std::endl;
	//std::cout << samples_after_dec_q << " &samples_after_dec_q" << std::endl;
	//std::cout << samples_after_dec_demod << " &samples_after_dec_demod" << std::endl;
	//std::cout << length_tail_i << " &length_tail_i" << std::endl;
	//std::cout << length_tail_q << " &length_tail_q" << std::endl;
	//std::cout << length_tail_demod << " &length_tail_demod" << std::endl;
	//memset(merge_arr_i, 0, hackrf_i_q_shift_size + length_tail_i);
	//conv_dec2(tail_i_debug, &length_tail_i, buffer_i,       length_buf,                   &ind,   dec_rate, fir_imp_koef, len_fir_imp, &length_out_i,        out_i_debug,   merge_arr_i);
	  //conv_dec2(tail_i,       &length_tail_i, hackrf_i_shift, hackrf_i_q_shift_size,        &ind_i, 10,       fir_imp_koef, len_fir_imp, &samples_after_dec_i, hackrf_i_dec1, merge_arr_i);
	  conv_dec2(tail_i_i,		  &length_tail_i, hackrf_i_shift, hackrf_i_q_shift_size, &ind_i, 10, fir_imp_koef, len_fir_imp, &samples_after_dec_i, hackrf_i_dec1, merge_arr_i);
	  for (int i = 0; i < 40; i++)
	  {
		  //std::cout << hackrf_i_dec1[i] << "=hackrf_i_dec1[" << i << "] rx_callback " << std::endl;
	  }
	 // std::cout << "             i          rx_callback            " << std::endl;
	//std::cout << samples_after_dec_i << " &samples_after_dec_i" << std::endl;
	//std::cout << length_tail_i << " &length_tail_i" << std::endl;
	//memset(tail_i_i, 0, length_tail_i*sizeof(double));
	//std::cout<<"convdec i done"<<std::endl;
	for (int i = 0; i < length_tail_i; i++)
	{
		//std::cout << tail_i_i[i] << "=tail_i[" << i << "] rx_callback   after memset" << std::endl;
	}
	//std::cout << "q1" << std::endl;
	//conv_dec1(hackrf_q_shift, hackrf_i_q_shift_size, fir_imp_koef, len_fir_imp, 10, &ind_q, hackrf_q_dec1, tail_q, &length_tail_q);
	//memset(merge_arr_q, 0, hackrf_i_q_shift_size + length_tail_q);
	//conv_dec2(tail_i_debug, &length_tail_i, buffer_i,       length_buf,            &ind,   dec_rate, fir_imp_koef, len_fir_imp, &length_out_i,        out_i_debug);
	  conv_dec2(tail_q,       &length_tail_q, hackrf_q_shift, hackrf_i_q_shift_size,		&ind_q, 10,       fir_imp_koef, len_fir_imp, &samples_after_dec_q, hackrf_q_dec1, merge_arr_q);
	  //std::cout<<"convdec q done"<<std::endl;
	  for (int i = 0; i < 40; i++)
	  {
		  //std::cout << hackrf_q_dec1[i] << "=hackrf_q_dec1[" << i << "] rx_callback " << std::endl;
	  }
	//  std::cout << "             q           rx_callback           " << std::endl;
	//std::cout << samples_after_dec_q << " &samples_after_dec_q" << std::endl;
	//std::cout << length_tail_q << " &length_tail_q" << std::endl;
	//conv_dec(hackrf_i_shift, hackrf_i_q_shift_size, fir_imp_koef, len_fir_imp, hackrf_i_dec1, 10, samples_after_dec_i);
	//conv_dec(hackrf_q_shift, hackrf_i_q_shift_size, fir_imp_koef, len_fir_imp, hackrf_q_dec1, 10, samples_after_dec_q);
	//std::cout << "demod" << std::endl;
	fm_demod(hackrf_i_dec1, hackrf_q_dec1, demod_samples, samples_after_dec_i);
	//std::cout<<"demod done"<<std::endl;
	//std::cout << length_in_demod << "=                   length_in_demod" << std::endl;
	//������ ������� ���������� � ��������� � 5 ���
	//conv_dec1(demod_samples, samples_after_dec, fir_imp_koef1, len_fir_imp, 10, &ind, demod_dec, tail_demod, &length_tail_demod);
	//memset(merge_arr_demod, 0, samples_after_dec_i + length_tail_demod);
	//conv_dec2(tail_i_debug, &length_tail_i,     buffer_i,      length_buf,          &ind,       dec_rate, fir_imp_koef,  len_fir_imp, &length_out_i,            out_i_debug);
	  conv_dec2(tail_demod1,   &length_tail_demod, demod_samples, samples_after_dec_i-1, &ind_demod, 10,       fir_imp_koef1, len_fir_imp, &samples_after_dec_demod, demod_dec, merge_arr_demod);
	  //std::cout<<"convdec demod done"<<std::endl;
	  for (int i = 0; i < 40; i++)
	  {
		 // std::cout << demod_samples[i] << "=demod_samples[" << i << "] rx_callback" << std::endl;
	  }
	//  std::cout << "             demod          rx_callback            " << std::endl;
	//std::cout << samples_after_dec_demod << " &samples_after_dec_demod" << std::endl;
	//std::cout << length_tail_demod << " &length_tail_demod" << std::endl;
	//conv_dec(demod_samples, samples_after_dec_i, fir_imp_koef1, len_fir_imp, demod_dec, 10, samples_after_dec_i); //1313
	//������� samples_after_dec_i � q
	//conv_dec(hackrf_q_dec1, samples_after_dec_q, fir_imp_koef1, len_fir_imp, hackrf_q_dec2, 5, samples_after_dec_q);
	
	////������ ������� ���������� � ��������� � 10 ���
	//conv(hackrf_i_shift, hackrf_i_q_shift_size, fir_imp_koef, len_fir_imp, hackrf_i_fir1);
	//conv(hackrf_q_shift, hackrf_i_q_shift_size, fir_imp_koef, len_fir_imp, hackrf_q_fir1);
	//decim(hackrf_i_fir1, hackrf_i_dec1, hackrf_i_q_fir1_size, 10, samples_after_dec_i);
	//decim(hackrf_q_fir1, hackrf_q_dec1, hackrf_i_q_fir1_size, 10, samples_after_dec_i);
	////������ ������� ���������� � ��������� � 5 ���
	//conv(hackrf_i_dec1, samples_after_dec_i, fir_imp_koef, len_fir_imp, hackrf_i_fir2);
	//conv(hackrf_q_dec1, samples_after_dec_i, fir_imp_koef, len_fir_imp, hackrf_q_fir2);
	//decim(hackrf_i_fir2, hackrf_i_dec2, hackrf_i_q_fir2_size, 5, samples_after_dec_i);
	//decim(hackrf_q_fir2, hackrf_q_dec2, hackrf_i_q_fir2_size, 5, samples_after_dec_i);

	//����������� �������������� ������� � �������� ������������� 24 ���
	//fm_demod(hackrf_i_dec2, hackrf_q_dec2, demod_samples, samples_after_dec_i);
	//std::cout << "accum_demod=                         " << accum_demod << std::endl;
	DC_remove_double(demod_dec, DC_shift, samples_after_dec_demod,&accum_demod);
	//std::cout<<"dc remove done"<<std::endl;
	//������ � ����
	//fwrite(demod_samples, sizeof(double), samples_after_dec_i, file_demod_snd);
	//��������� ������ � ���������� � ������� �� ������� (������������ ���������� ������ �� 127 ?)
	double max=0;
	for (int i = 0; i < samples_after_dec_demod; i++) {
		if (abs(demod_dec[i]) > max) {
			max = abs(demod_dec[i]);
		}
	}
	//std::cout<<"max found"<<std::endl;
	//std::cout << max << "max" << std::endl;
	//if (max < 2) { max = 32767; }
	//std::cout << "max=" << max << std::endl;
	for (int i = 0; i < samples_after_dec_demod; i++) {
		//sndDinBuffer.Push(ATOMIC_QUEUE_TYPE(demod_dec[i] / max * 32767));
		demod_dec_int[i]=(int16_t)(demod_dec[i]/max*32767);
	}
	//std::cout<<"normalization done"<<std::endl;
	player_buffer.write(demod_dec_int,samples_after_dec_demod*2);
	//std::cout<<"write done"<<std::endl;
	return 0;
}
int main() {
	int reset_flag = 0;
	shift_freq_hz = -192000;
	//iq=fopen("iq.cs8","wb");
	//FILE* sinus=fopen("sin.int16","rb");
	//int16_t sin_buf[24000];
	//fread(sin_buf,sizeof(int16_t),24000,sinus);
    
	// struct sigaction sa;
    // sa.sa_handler = signal_handler;
    // sigemptyset(&sa.sa_mask);
    // sa.sa_flags = SA_RESTART; // Перезапуск системных вызовов
	// set_nonblocking_mode(true);

    // // Запускаем поток для обработки ввода
    // std::thread input(input_thread);
    // if (sigaction(SIGINT, &sa, nullptr) == -1) {
    //     std::cerr << "Ошибка установки обработчика сигнала!" << std::endl;
    //     return 1;
    // }

	signal(SIGINT, signal_handler);
    //signal(SIGTERM, signal_handler);
	load_fir_imp((char*)"fir_imp2.txt", len_fir_imp, fir_imp_koef);
	load_fir_imp((char*)"filter3.txt", len_fir_imp, fir_imp_koef1);

	//recorder_buffer.write(sin_buf,13000);
    recorder_thread = std::thread(record_thread_func);
   	player_thread = std::thread(playback_thread_func);
	//std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    std::cout << "hackrf_init=				" << hackrf_error_name((hackrf_error)hackrf_init()) << std::endl;
	std::cout << "hackrf_open=				" << hackrf_error_name((hackrf_error)hackrf_open(&device)) << std::endl;


	std::cout << "hackrf_set_sample_rate=			" << hackrf_error_name((hackrf_error)hackrf_set_sample_rate(device, 2400000)) << std::endl;
	std::cout << "hackrf_set_freq=			" << hackrf_error_name((hackrf_error)hackrf_set_freq(device, FREQ)) << std::endl;
	std::cout << "hackrf_set_txvga_gain=			" << hackrf_error_name((hackrf_error)hackrf_set_txvga_gain(device, 30)) << std::endl; //30
	std::cout << " hackrf_set_lna_gain=			" << hackrf_error_name((hackrf_error)hackrf_set_lna_gain(device, 8)) << std::endl;	//IF ������ �����
	std::cout << "hackrf_set_vga_gain=			" << hackrf_error_name((hackrf_error)hackrf_set_vga_gain(device, 36)) << std::endl;  //BB ������ �����
	std::cout << "hackrf_set_amp_enable=			" << hackrf_error_name((hackrf_error)hackrf_set_amp_enable(device, 0)) << std::endl;
    //std::cout << "starting tx..." << std::endl;


	//std::cout << "hackrf_start_tx=			" << hackrf_error_name((hackrf_error)hackrf_start_tx(device, fm_tx_callback, NULL)) << std::endl;
	//std::cout << "hackrf_set_freq=			" << hackrf_error_name((hackrf_error)hackrf_set_freq(device, FREQ)) << std::endl;


	std::cout << "hackrf_start_rx=			" << hackrf_error_name((hackrf_error)hackrf_start_rx(device, rx_callback, NULL)) << std::endl;
	std::cout << "hackrf_set_freq=			" << hackrf_error_name((hackrf_error)hackrf_set_freq(device, FREQ+shift_freq_hz)) << std::endl;
    std::cout << "Запуск записи и воспроизведения... Нажмите Ctrl+C для остановки.\n";


	// while(mode_switcher.get_mode()==0){
	// 	//
	// 	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	// 	//recorder_buffer.write(sin_buf,13000);
	// 	//std::cout<<"recorder_buffer.Size()= "<<recorder_buffer.Size()<<std::endl;
	// 	std::cout<<"                                                player_buffer.Size()= "<<player_buffer.Size()<<std::endl;
	// };

while (!stop_flag) {
		// ������� �� 1 �������
		//Sleep(1000);
		sleep(1);
		if ((mode_switcher.get_mode() == 1) && (transceiver_mode == TRANSCEIVER_MODE_RX)) //tx mode
		{	
			//player.stop();
			//recorder.start();
			recorder_buffer.clear();
			hackrf_stop_rx(device);
			hackrf_start_tx(device, fm_tx_callback, NULL);
			hackrf_set_freq(device, FREQ);
			transceiver_mode = TRANSCEIVER_MODE_TX;
			std::cout << "TX MODE" << std::endl;
			reset_flag = 0;
		}
		else if ((mode_switcher.get_mode() == 0) && (transceiver_mode == TRANSCEIVER_MODE_TX)) //rx mode
		{
			for (int i = 0; i < 4000; i++)
			{
				//sndDinBuffer.Push(0);
			}
			//player.start();
			//recorder.stop();
			hackrf_stop_tx(device);
			hackrf_start_rx(device, rx_callback, NULL);
			hackrf_set_freq(device, FREQ + shift_freq_hz);
			transceiver_mode = TRANSCEIVER_MODE_RX;
			std::cout << "RX MODE" << std::endl;
			reset_flag = 0;
		}
		else if (hackrf_is_streaming(device)==-1003)
		{	
			std::cout << "restarting the hackrf..." << std::endl;
			std::cout << "hackrf functions status:" << std::endl;
			if (reset_flag == 0)
			{
				std::cout << "hackrf_reset=				" << hackrf_error_name((hackrf_error)hackrf_reset(device)) << std::endl;
				reset_flag = 1;
			}
			std::cout << "hackrf_init=				" << hackrf_error_name((hackrf_error)hackrf_init()) << std::endl;
			std::cout << "hackrf_open=				" << hackrf_error_name((hackrf_error)hackrf_open(&device)) << std::endl;
			std::cout << "hackrf_set_sample_rate=			" << hackrf_error_name((hackrf_error)hackrf_set_sample_rate(device, 2400000)) << std::endl;
			std::cout << "hackrf_set_txvga_gain=			" << hackrf_error_name((hackrf_error)hackrf_set_txvga_gain(device, 30)) << std::endl; //30
			std::cout << " hackrf_set_lna_gain=			" << hackrf_error_name((hackrf_error)hackrf_set_lna_gain(device, 8)) << std::endl;	//IF ������ �����
			std::cout << "hackrf_set_vga_gain=			" << hackrf_error_name((hackrf_error)hackrf_set_vga_gain(device, 36)) << std::endl;  //BB ������ �����
			std::cout << "hackrf_set_amp_enable=			" << hackrf_error_name((hackrf_error)hackrf_set_amp_enable(device, 0)) << std::endl;
			if (transceiver_mode == TRANSCEIVER_MODE_RX)
			{
				std::cout << "starting rx..." << std::endl;
				std::cout << "hackrf_start_rx=			" << hackrf_error_name((hackrf_error)hackrf_start_rx(device, rx_callback, NULL)) << std::endl;
				std::cout << "hackrf_set_freq=			" << hackrf_error_name((hackrf_error)hackrf_set_freq(device, FREQ)) << std::endl;
			}
			else
			{
				recorder_buffer.clear();
				std::cout << "starting tx..." << std::endl;
				std::cout << "hackrf_start_tx=			" << hackrf_error_name((hackrf_error)hackrf_start_tx(device, fm_tx_callback, NULL)) << std::endl;
				std::cout << "hackrf_set_freq=			" << hackrf_error_name((hackrf_error)hackrf_set_freq(device, FREQ + shift_freq_hz)) << std::endl;
			}
		}
		if (transceiver_mode == TRANSCEIVER_MODE_RX) {
			std::cout << "\t" << "\t" << "\t" << "\t" << "sndDinBuffer.Size()= " << player_buffer.Size() << std::endl;
			std::cout << "sndMicBuffer.Size()= " << recorder_buffer.Size() << std::endl;
			std::cout << std::endl;
			std::cout << "avg buffer level: " << avg_level(hackrf_i,hackrf_q);
			std::cout << std::endl;
			recorder_buffer.clear();
		}
		if (transceiver_mode == TRANSCEIVER_MODE_TX) {
			//const int length = 48000;
			//file_read_short_to_atomic(length, (char*)"../mathcad/sin.bin");
			std::cout << "\t" << "\t" << "\t" << "\t" << "sndMicBuffer.Size()= " << recorder_buffer.Size() << std::endl;
			std::cout << "sndDinBuffer.Size()= " << player_buffer.Size() << std::endl;
		}
		std::cout << "   hackrf_is_streaming        " << hackrf_error_name((hackrf_error)hackrf_is_streaming(device)) << std::endl;
		//std::cout << mode_switcher.get_mode() << std::endl;
	}

	//std::cin.get();
	std::cout << "Нажмите Ctrl+C для остановки.\n";
	std::cout<<"hackrf_stop_tx= "<<hackrf_error_name((hackrf_error)hackrf_stop_tx(device)) <<std::endl;	
	std::cout<<"hackrf_stop_rx= "<<hackrf_error_name((hackrf_error)hackrf_stop_rx(device)) <<std::endl;	
	std::cout<<"hackrf_close="<<hackrf_error_name((hackrf_error)hackrf_close(device)) <<std::endl;
	std::cout<<"hackrf_exit"<<hackrf_error_name((hackrf_error)hackrf_exit()) <<std::endl;
	//fflush(sinus);
	//fclose(sinus);
	recorder_buffer.stop(); // Останавливаем буфер
    player_buffer.stop();
	force_stop_threads();
	player_thread.join();
    recorder_thread.detach(); // Ждём завершения потоков
 
    
    return 0;
}