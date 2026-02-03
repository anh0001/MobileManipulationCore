# Kubernetes Deployment (Future)

This directory will contain Kubernetes manifests for deploying MobileManipulationCore at scale.

## Use Cases

- Multi-robot fleet coordination
- Cloud-based policy inference
- Load-balanced policy servers
- Monitoring and observability

## Planned Structure

```
k8s/
├── namespace.yaml           # Dedicated namespace
├── policy-deployment.yaml   # Policy server deployment
├── policy-service.yaml      # Service for policy endpoints
├── policy-hpa.yaml         # Horizontal Pod Autoscaler
├── configmap.yaml          # Configuration
├── secrets.yaml            # Sensitive data
├── ingress.yaml            # External access
└── monitoring/             # Prometheus, Grafana
```

## Coming Soon

Full Kubernetes deployment will be added in a future release.

For now, use Docker Compose for orchestration.

## References

- [Kubernetes Best Practices](https://kubernetes.io/docs/concepts/configuration/overview/)
- [ROS on Kubernetes](https://github.com/ros-infrastructure/kubernetes)
