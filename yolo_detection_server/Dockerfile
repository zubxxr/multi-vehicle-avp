FROM nvidia/cuda:12.6.0-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install system dependencies
RUN apt update && apt install -y \
    python3 \
    python3-pip \
    git \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Upgrade pip first (separate layer for caching)
RUN pip3 install --upgrade pip

# Copy only requirements first for better Docker caching
COPY requirements.txt .

RUN pip3 install --no-cache-dir -r requirements.txt

# Copy application files explicitly
COPY yolo_server.py .
COPY weight.pt .

EXPOSE 5000

CMD ["python3", "yolo_server.py"]