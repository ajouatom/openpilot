FROM python:3.12-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

RUN useradd --create-home --uid 10001 carrot \
    && mkdir -p /data/openpilot /data/state \
    && chown -R carrot:carrot /data
WORKDIR /app
COPY requirements.txt ./
RUN pip install --no-cache-dir --requirement requirements.txt
COPY --chown=carrot:carrot __init__.py server.py ./
RUN chmod 0755 /app \
    && chmod 0644 /app/__init__.py /app/server.py

USER carrot
EXPOSE 8080
CMD ["python", "server.py"]
