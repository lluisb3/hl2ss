FROM python:3.9

USER root

RUN apt-get update

RUN apt-get install -y libgl1

RUN mkdir -p /home/app

RUN mkdir -p /home/app/data

COPY . /home/app

WORKDIR /home/app

RUN chmod u+rwx viewer/*

RUN pip install -r requirements.txt

CMD [ "/bin/bash" ]