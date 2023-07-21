FROM ubuntu:jammy

RUN apt-get update && apt-get install -y \
    sudo \
    cmake \
    git \
    build-essential \
    liblua5.2-dev \
    libboost-all-dev \
    libprotobuf-dev \
    libtbb-dev \
    libstxxl-dev \
    libbz2-dev

WORKDIR /data

COPY . /data

RUN /data/compile.sh
RUN /data/build.sh

ENTRYPOINT ["/data/build/data-generator"]
