# Use an official Python runtime as a parent image
FROM python:2.7-slim

# RUN UPDATES FOR UBUNTU
RUN apt-get update && apt-get install -y cppcheck libgmp3-dev

# Copy the current directory contents into the container at /app
COPY src /src
COPY requirements.txt /requirements.txt

WORKDIR /

# Install packages specified in requirements.txt
RUN pip install --trusted-host pypi.org --trusted-host files.pythonhosted.org --trusted-host pypi.python.org -r requirements.txt

RUN python -m nltk.downloader -d /usr/share/nltk_data wordnet

WORKDIR /src

# Run app.py when the container launches
ENV LD_LIBRARY_PATH /pgm
ENTRYPOINT ["python", "./prob_phys_units.py"]


