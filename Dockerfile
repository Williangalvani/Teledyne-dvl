FROM python:3.9-slim-bullseye

RUN apt update && apt install -y nmap build-essential cmake

# Create default user folder
RUN mkdir -p /home/pi

RUN pip3 install requests loguru pymavlink pyserial numpy wheel --extra-index-url https://www.piwheels.org/simple
RUN apt install -y libopenblas-dev

# Install dvl service
COPY wayfinder /home/pi/wayfinder
RUN cd /home/pi/wayfinder && pip3 install .

LABEL version="1.0.0"
LABEL permissions='\
{\
    "NetworkMode": "host"\
}'
LABEL authors='[\
    {\
        "name": "Willian Galvani",\
        "email": "willian@bluerobotics.com"\
    }\
]'
LABEL company='{\
        "about": "",\
        "name": "Blue Robotics",\
        "email": "support@bluerobotics.com"\
    }'
LABEL type="device-integration"
LABEL tags='[\
        "positioning",\
        "navigation",\
        "doppler-velocity-log"\
    ]'
LABEL readme='https://raw.githubusercontent.com/williangalvani/Teledyne-dvl/{tag}/README.md'
LABEL links='{\
        "website": "https://raw.githubusercontent.com/williangalvani/Teledyne-dvl",\
        "support": "https://raw.githubusercontent.com/williangalvani/Teledyne-dvl/issues"\
    }'
LABEL requirements="core >= 1.0"

RUN pip3 install flask

ENTRYPOINT /home/pi/wayfinder/integration/main.py
