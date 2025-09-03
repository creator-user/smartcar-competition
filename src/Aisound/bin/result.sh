
# 进入tts_offline_sample目录
cd ~/Desktop/ucar/src/Aisound/samples/tts_offline_sample/

pwd

# 编译
./64bit_make.sh 

# 进入bin目录
cd ~/Desktop/ucar/src/Aisound/bin/

# 生成音频
./tts_offline_sample

# 播放音频
play tts_sample.wav 
