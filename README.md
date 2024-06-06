# Object Detection

## Objective

This project sets up a ROS 2 Humble-based object detection system using YOLOv5 in Docker and deploys it on Kubernetes.

## Approach

* **OS**
We will work on Ubuntu Jammy (22.04)
* **Environment** 
Docker
* **Deployment** 
Kubernetes
* **Middleware**
ROS2 Humble


## Setup Instructions

Follow the steps below to set up and run the project.

### Prerequisites

- Docker
- Kubernetes
- ROS 2 Humble

### Steps 

**1. Clone the repository**
```
$ git clone https://github.com/alexandrosnic/object-detection.git
$ cd object-detection
```

**2. Start Mnikube**
```
$ minikube start
```

**3. Build the docker image**
```
docker build -t object-detection .
```

**4. Apply deployment**
```
kubectl apply -f deployment.yaml
```

Or run each node independently:

**5. Expose service**
```
kubectl expose deployment object-detection-deployment --type=NodePort --port=8080
```
