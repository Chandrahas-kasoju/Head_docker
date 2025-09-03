#!/bin/bash
git clone https://github.com/hailo-ai/hailo-rpi5-examples.git
cd hailo-rpi5-examples
source setup_env.sh
pip install -r requirements.txt
./download_resources.sh