# Slim version of Python
FROM python:3.8-slim-buster

# Download Package Information
RUN apt-get update -y

# Install Tkinter
RUN apt-get install tk -y

# Set working directory
WORKDIR /app

# Copy python file requirements
COPY requirements.txt requirements.txt

#
RUN pip3 install -r requirements.txt

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/London
RUN apt-get install -y python3-tk

COPY . .
#CMD [ "python3", "-m" , "terrestrial_planner", "--host=0.0.0.0"]
#CMD ["/app/terrestrial_planner.py"]
#ENTRYPOINT ["python3"]
