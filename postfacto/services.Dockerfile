FROM ghcr.io/astral-sh/uv:python3.12-bookworm-slim
ARG SCRIPT_FILE

COPY . /app
WORKDIR /app
ENV UV_COMPILE_BYTECODE=1

RUN uv sync --locked

CMD uv run --env-file config $SCRIPT_FILE
