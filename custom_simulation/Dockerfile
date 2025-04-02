# Use an official Python runtime as a parent image
FROM python:3.11

# Set the working directory in the container
WORKDIR /app

# Copy the current directory contents into the container
COPY . .

# Install any dependencies (if you have a requirements.txt)
RUN pip install --no-cache-dir -r requirements.txt

# Set the command to run the application
CMD ["python", "main.py"]