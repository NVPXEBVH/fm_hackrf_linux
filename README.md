# fm_hackrf_linux
кароче фм радиостанция под линукс библиотека для звука asound маленько прерывается звук на динамике маленько на передаче тоже скорее всего из за специфики работы с бибилиотекой и кароче не всегда выходит из потока рекордера нужно просто закрывать терминал
скачать библиотеки
sudo apt-get install libhackrf-dev
sudo apt-get install libasound-dev
//компиляция
g++ main.cpp buffer.cpp audio_utils.cpp  Mode_switcher.cpp -o rec_play -lasound -lhackrf -lm
//запуск 
./rec_play
