---
title: "Service"
permalink: /docs/service/
excerpt: "AutoMiny systemd service"
toc: true
---

The AutoMiny stack comes preinstalled as a systemd service. It is launched automatically on system start after WiFi becomes available. In case you want to restart, stop, start or view the logs you can use the following commands:

### Stopping the service
```bash
sudo systemctl stop autominy
```

### Starting the service
```bash
sudo systemctl start autominy
```

### Restarting the service
```bash
sudo systemctl restart autominy
```

### View the logs
```bash
journalctl -u autominy
```