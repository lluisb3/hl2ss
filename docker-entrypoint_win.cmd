# docker-entrypoint_linux.cmd

set IMAGE=lluisb3/hl2ss:v7.0
set IP_HOLOLENS=192.168.1.107
set EXP_NAME=hololens_docker


docker run -it --rm -v ~/docker_volume/hl2ss/data:/home/app/data %IMAGE% \
python3 viewer/advanced_recorder.py --exp_name %EXP_NAME% --ip_hololens %IP_HOLOLENS%
