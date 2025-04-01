# Copy this code into ~/.bashrc
python-background() {
    sudo nohup python3 "$@" >> /path/to/your/log.log 2>&1 &
}
