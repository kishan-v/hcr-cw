FROM python:3.9-slim

WORKDIR /app

COPY fake_data.py .
COPY sender.py .
COPY lidar_packer.py .

CMD ["python", "sender.py"]
