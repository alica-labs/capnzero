# Cap'n Zero

A communication middleware based on [Cap'n Proto](capnproto.org) and [ZeroMQ](zeromq.org). It runs under Ubuntu 18.04 and utilises C++ Standard 11 and maybe higher.

## Install Cap'n Proto

    sudo apt install libcapnp-dev
    
## Install Cap'n Proto

    sudo su
    echo "deb http://download.opensuse.org/repositories/network:/messaging:/zeromq:/git-draft/xUbuntu_18.04/ ./" >> /etc/apt/sources.list
    exit
    wget https://download.opensuse.org/repositories/network:/messaging:/zeromq:/git-draft/xUbuntu_18.04/Release.key -O- | sudo apt-key add
    sudo apt install libzmq3-dev

