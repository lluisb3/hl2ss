FROM ubuntu:22.04

# Not cache pip packages
RUN echo "[install]\ncompile = no\n\n[global]\nno-cache-dir = True" > /etc/pip.conf

# Set timezone to not be interactive the python installation
ENV TZ=Europe/Madrid

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Set non-root user as user
ARG USER_ID
ARG GROUP_ID

RUN apt-get update && \
    apt-get install -y sudo && \
    addgroup --gid $GROUP_ID user && \
    adduser --uid $USER_ID --gid $GROUP_ID --disabled-password --gecos "Default user" user && \
    echo 'user ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers

USER user

# Install necesary libraries
RUN sudo apt-get install -y libgl1 libglib2.0-0

# Install Python and git
RUN sudo apt-get install -y python3.9 python3-pip git-all

ENV PATH=/home/user/.local/bin:$PATH

# Creat hl2ss app folder
RUN mkdir -p /home/user/app

RUN mkdir -p /home/user/app/data

# Set workdirectory to hl2ss app
WORKDIR /home/user/app

# Copy files and set user ownership of the files in app folder
COPY --chown=user:user . /home/user/app
RUN chmod -R u+rwx /home/user/app

# Install necesary requeriments
RUN pip3 install -r requirements.txt

# Run bash as entrypoint
CMD [ "/bin/bash" ]