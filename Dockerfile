FROM ubuntu:16.04

COPY src /src
COPY requirements.txt /requirements.txt

RUN apt-get update && apt-get install -y cppcheck libgmp3-dev python-pip 
# UPGRADE PIP TO LAST VERSION TO WORK WITH PYTHON 2.7
RUN /usr/bin/env python -m pip install --upgrade pip==20.3.3
RUN pip install --trusted-host pypi.org --trusted-host files.pythonhosted.org --trusted-host pypi.python.org -r requirements.txt
RUN python -m nltk.downloader -d /usr/share/nltk_data wordnet


WORKDIR /src
ENTRYPOINT ["python", "./prob_phys_units.py"]
#CMD ["bash"]
