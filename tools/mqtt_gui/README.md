# MQTT Control GUI

Simple Python desktop GUI for your Dreimaskin MQTT topics.

## Features

- Connect/disconnect to MQTT broker
- Set target position (`dreimaskin_els/command`)
- Set motion mode (`dreimaskin_els/command/mode`)
- Live view of:
  - position
  - speed
  - target
  - distance to target
  - mode
  - alive/status
- Small event log panel

## Install

From this folder:

```powershell
pip install -r requirements.txt
```

## Run

```powershell
python app.py
```

## Default topics

- Command target: `dreimaskin_els/command`
- Command mode: `dreimaskin_els/command/mode`
- Status position: `dreimaskin_els/status/position`
- Status speed: `dreimaskin_els/status/speed`
- Status target: `dreimaskin_els/status/target`
- Status distance to target: `dreimaskin_els/status/distance_to_target`
- Status mode: `dreimaskin_els/status/mode`
- Status alive: `dreimaskin_els/status`
