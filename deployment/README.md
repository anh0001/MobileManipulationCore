# Deployment

This directory contains deployment configurations for MobileManipulationCore.

## Directory Structure

```
deployment/
├── docker/              # Docker configurations
│   ├── Dockerfile.jetson       # For Jetson robot
│   ├── Dockerfile.policy       # For policy server
│   ├── docker-compose.yml      # Orchestration
│   ├── entrypoint.sh          # Jetson entrypoint
│   └── entrypoint_policy.sh   # Policy server entrypoint
└── k8s/                # Kubernetes manifests (future)
```

## Deployment Profiles

### Profile A: Jetson-Only (On-Device)

Everything runs on the Jetson AGX Orin.

**Setup:**
```bash
# On Jetson
cd ~/manipulation_ws
source install/setup.bash
ros2 launch manipulation_bringup core_launch.py
```

**Docker (optional):**
```bash
docker build -t manipulation-jetson -f deployment/docker/Dockerfile.jetson .
docker run --runtime nvidia --network host -it manipulation-jetson
```

### Profile B: Split Deployment (Jetson + Remote Server)

Policy inference on remote GPU server, control on Jetson.

**Step 1: Start policy server (on GPU machine)**
```bash
# Using Docker Compose (recommended)
cd deployment/docker
docker-compose up policy-server

# Or build and run manually
docker build -t manipulation-policy -f Dockerfile.policy ../..
docker run --gpus all -p 5000:5000 \
  -v ./models:/app/models \
  manipulation-policy
```

**Step 2: Start robot stack (on Jetson)**
```bash
# Native (recommended)
ros2 launch manipulation_bringup core_launch.py \
  use_remote_policy:=true \
  remote_url:=http://<SERVER_IP>:5000

# Or with Docker
docker run --runtime nvidia --network host \
  -e POLICY_SERVER_URL=http://<SERVER_IP>:5000 \
  manipulation-jetson \
  ros2 launch manipulation_bringup core_launch.py \
    use_remote_policy:=true \
    remote_url:=$POLICY_SERVER_URL
```

## Docker Images

### Building

**Jetson image:**
```bash
cd /path/to/MobileManipulationCore
docker build -t manipulation-jetson:v0.1.0 \
  -f deployment/docker/Dockerfile.jetson .
```

**Policy server image:**
```bash
docker build -t manipulation-policy:v0.1.0 \
  -f deployment/docker/Dockerfile.policy .
```

### Running

**Policy server with GPU:**
```bash
docker run --gpus all \
  -p 5000:5000 \
  -v $(pwd)/models:/app/models \
  manipulation-policy:v0.1.0
```

**Jetson container:**
```bash
docker run --runtime nvidia \
  --network host \
  --privileged \
  -v /dev:/dev \
  manipulation-jetson:v0.1.0
```

## Docker Compose

For coordinated deployment:

```bash
cd deployment/docker

# Start all services
docker-compose up

# Start in background
docker-compose up -d

# View logs
docker-compose logs -f

# Stop
docker-compose down
```

### Environment Variables

Configure via `.env` file in `deployment/docker/`:

```bash
# .env file example
ROS_DOMAIN_ID=0
CUDA_VISIBLE_DEVICES=0
POLICY_MODEL_NAME=openvla-7b
INFERENCE_RATE=10.0
```

## Security

### SROS2 (ROS 2 Security)

For production deployments with split configuration:

1. **Generate keys:**
   ```bash
   ros2 security create_keystore /path/to/keystore
   ros2 security create_key /path/to/keystore /policy_node
   ros2 security create_key /path/to/keystore /adapter_node
   ```

2. **Enable in launch:**
   ```bash
   export ROS_SECURITY_KEYSTORE=/path/to/keystore
   export ROS_SECURITY_ENABLE=true
   ros2 launch manipulation_bringup core_launch.py
   ```

### VPN Setup

For remote policy server over internet:

1. **Install WireGuard** (recommended)
   ```bash
   # On both Jetson and server
   sudo apt install wireguard
   ```

2. **Configure tunnel** between Jetson and server

3. **Update remote_url** to use VPN IP

## Kubernetes (Future)

For multi-robot deployments or cloud-based policy servers:

```
deployment/k8s/
├── namespace.yaml
├── policy-deployment.yaml
├── policy-service.yaml
└── ingress.yaml
```

See `k8s/README.md` for details (coming soon).

## Monitoring

### Docker Stats

```bash
# Resource usage
docker stats manipulation-policy-server

# Logs
docker logs -f manipulation-policy-server
```

### Health Checks

Policy server:
```bash
curl http://localhost:5000/health
```

## Troubleshooting

**Issue: GPU not detected in container**
- Ensure NVIDIA Container Toolkit is installed
- Check: `docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi`

**Issue: ROS 2 nodes can't discover each other**
- Use `--network host` on Jetson
- Check `ROS_DOMAIN_ID` matches
- Firewall: allow UDP ports 7400-7500

**Issue: Model download fails**
- Check HuggingFace access token if using gated models
- Verify network connectivity
- Pre-download models: `huggingface-cli download <model-id>`

**Issue: High latency in split mode**
- Reduce image resolution in config
- Enable image compression
- Check network bandwidth: `iperf3`

## Best Practices

1. **Jetson Performance**
   - Set max power mode: `sudo nvpmodel -m 0`
   - Boost clocks: `sudo jetson_clocks`
   - Monitor: `sudo tegrastats`

2. **Model Optimization**
   - Use FP16 for Jetson
   - Consider TensorRT conversion for production
   - Cache models locally

3. **Network**
   - Use wired connection for reliability
   - QoS settings for real-time topics
   - Latency < 100ms recommended

4. **Security**
   - Never expose policy server to public internet without auth
   - Use SROS2 for production
   - Rotate keys regularly

## References

- [Docker Documentation](https://docs.docker.com/)
- [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-docker)
- [SROS2](https://design.ros2.org/articles/ros2_dds_security.html)
- [Kubernetes](https://kubernetes.io/docs/)

---

*For development, native installation is often easier. Use Docker for production deployments.*
