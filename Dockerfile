FROM python:3.9

RUN mkdir -p /home/app

COPY . /home/app

WORKDIR /home/app

RUN pip install -r requirements.txt

CMD [ "python", "viewer/advanced_recorder_pcd_mesh.py" ]