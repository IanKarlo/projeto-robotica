FROM python:3.8-slim-bullseye

RUN mkdir /app
COPY ./requirements.txt /app/requirements.txt
COPY ./app/pyproject.toml /app/pyproject.toml

RUN apt-get update -y && apt-get install -y vim git make gcc g++

# . means /app
RUN pip install -r /app/requirements.txt

COPY --chmod=755 ./entry.sh /app/entry.sh
WORKDIR /app



