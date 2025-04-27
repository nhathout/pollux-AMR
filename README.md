# Pollux-AMR · Autonomous Counter-Top Cleaning Robot
![Maintained Badge](https://img.shields.io/badge/status-active-brightgreen)

> **Last verified:** 27 Apr 2025  
> **Target HW:** Raspberry Pi 4 Model B (4 GB)  
> **Target OS:** Ubuntu 20.04 LTS (headless) with ROS Noetic  

Pollux-AMR is a ROS-driven mobile robot that patrols a kitchen countertop, disinfecting
the surface with a UV-C LED strip while staying clear of cliffs and obstacles.
The original project used a rules-based brain; we now train a real-time PPO agent
on-board via **rl_brain_node.py** for continuous improvement.

---

## 1 · TL;DR Boot-Up  
    # ① Flash Ubuntu 20.04 to the µSD card, boot once to expand the FS
    # ② SSH in and run:
    git clone https://github.com/<your-org>/pollux-AMR.git
    cd pollux-AMR
    chmod +x setup_env.sh && ./setup_env.sh
    # ③ Reboot (systemd service now auto-starts every node)

*Need only a demo video?*
- Links to video demonstrations of Pollux's major functionalities (UV indicator LED, cliff detection, and obstacle avoidance):
    - **UV indication & cliff detection:** [click here](https://drive.google.com/file/d/1kTDrHVp9VE7UjMdm_vTjpf8ZEP2LUoZp/view?usp=sharing)
    - **Obstacle Avoidance:** [click here](https://drive.google.com/file/d/1ax3cWRlPb4nttXV8eBDei8ND6SfjERsN/view?usp=sharing)<br><br>

---

## 2 · Current State (Apr 2025)

| Sub-system | Status | Gotchas & Quirks |
|------------|--------|------------------|
| Sensor I/O (hw_publisher.py & hw_publisher_2.py) | ✅ stable | Uses **BCM** pin numbers; wrong ECHO wiring can fry GPIOs. |
| Motor driver (motor_cmd_node.py) | ✅ stable | H-bridge inrush >1 A; keep a 2 A buck or Pi browns out. |
| LED safety (led_gyro_node.py) | ✅ stable | IMU IRQ sometimes locks; power-cycle to clear. |
| RL brain (rl_brain_node.py) | 🟡 beta | Seems to work and learn; first 100 s policy is random. |
| Service file (pollux-robot.service) | ✅ stable | Update **WorkingDirectory=** if you move the repo. |

---

## 3 · Things We Wish We Knew Earlier
1. Ultrasonic crosstalk → fire downward sensors 50 ms apart.  
2. RAM-starved RL → set SWAP = 1024 MB.  
3. Clock drift hurts reward timer → run *timedatectl set-ntp true*.  
4. Wi-Fi sleep kills SSH → set *wifi.powersave = 2* in NetworkManager.  
5. The best power supply for the job.

---

## 4 · Recommended Next Steps

| Area | Content |
|------|------------------|
| Mechanical | Switch to 12 V steppers for torque & battery life. |
| CV / DL | Add Pi Cam 3 + YOLO-Nano for obstacle detection. |
| DevOps | Stream TensorBoard via MQTT → Grafana. |
| Research | 24 h on-board RL study → short research paper. |

---

## 5 · Directory Map
    pollux-AMR/
    ├─ hardware/
    ├─ images/
    ├─ nodes/
    │   ├─ hw_publisher.py
    │   ├─ hw_publisher_2.py
    │   ├─ motor_cmd_node.py
    │   ├─ led_control_node.py
    │   ├─ led_gyro_node.py
    │   └─ rl_brain_node.py
    ├─ launch/
    ├─ models/
    ├─ scripts/
    └─ systemd/

---

## 6 · Further Reading  
Software stack → README_SOFTWARE.md  
Hardware build → README_HARDWARE.md  

User Manual → [click here](https://docs.google.com/document/d/1WDNQtBAzQioiVHYzoj2xVuRMR3jV1yGgZe9JvNC5dmk/edit?usp=sharing)<br><br>

Final Test Plan & Report → [click here](https://docs.google.com/document/d/1nLfvciRge8nUAEQmeBG3rHHYXOoPHTD_1vJB__G8N-4/edit?usp=sharing)

*Winning* Shark Tank Video → [click here](https://drive.google.com/file/d/1ElmvJo_tTRmPK1KgQrT9eFHeo6qOJTg3/view?usp=sharing)

Critical Design Review (CDR) Presentation → [click here](https://docs.google.com/presentation/d/1PCzo8z48-ifrknA4HLdUseuIMsqwAZrbvwXN47jLewY/edit?usp=sharing)

Shark Tank Presentation → [click here](https://docs.google.com/presentation/d/1J7x6Hkm6MfULzXxtW8gPblj4HMrZ-OuL5eKAxO-fepI/edit?usp=sharing)

2nd Prototype Test Report → [click here](https://docs.google.com/document/d/1qSaTDLb16L1KKv8wPwOk-qygIFx6U2V5G-Wa-tfIYAA/edit?usp=sharing)

1st Prototype Test Report → [click here](https://docs.google.com/document/d/1bSMtuGWDiYZbLjaCkalLF-rUq3OvFMxzeYqiuovF3EY/edit?usp=sharing)

Preliminary Design Review (PDR) Presentation → [click here](https://docs.google.com/presentation/d/1AgygkBsC4uNbJwVAmwFr3u0lZS7qua3f1DQ_0c6GlmY/edit?usp=sharing)

---

## 7 · License  
MIT License — © 2025 Pollux-AMR contributors