FROM ubuntu:24.04

RUN apt-get update && \
    apt-get install -y --no-install-recommends clang-format-18 ca-certificates findutils bash && \
    rm -rf /var/lib/apt/lists/*

ARG UID=1000
ARG GID=1000

WORKDIR /workspace

USER ${UID}:${GID}

CMD ["bash"]