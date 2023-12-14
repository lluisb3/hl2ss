
sudo docker run -it --rm -v ~/docker_volume/hl2ss/data:/home/app/data hl2ss:v1.0 \
python3 viewer/advanced_recorder_win.py --exp_name hololens_docker --ip_hololens 192.168.1.104
