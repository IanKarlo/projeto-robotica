FROM python:3.7.17-slim-bullseye

# Instale tudo que o Poetry precisa
RUN apt-get update -y && apt-get install -y \
    build-essential \
    libffi-dev \
    libssl-dev \
    zlib1g-dev \
    libbz2-dev \
    libreadline-dev \
    libsqlite3-dev \
    wget \
    curl \
    llvm \
    libncursesw5-dev \
    xz-utils \
    tk-dev \
    libxml2-dev \
    libxmlsec1-dev \
    liblzma-dev \
    python3-venv \
 && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv .virenv
RUN .virenv/bin/pip install -U pip setuptools
RUN .virenv/bin/pip install poetry
RUN ln -s /.virenv/bin/poetry /usr/local/bin/poetry

WORKDIR /app

# Copie antes para aproveitar o cache
COPY pyproject.toml poetry.lock poetry.toml .
RUN poetry install --no-interaction --no-ansi


COPY . .
