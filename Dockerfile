FROM ubuntu:bionic

# Prepare the build environment and dependencies
RUN \
	set -x && \
	apt-get update && \
	apt-get -y install software-properties-common && \
	add-apt-repository ppa:team-gcc-arm-embedded/ppa && \
	add-apt-repository ppa:jonathonf/tup && \
	apt-get update && \
	apt-get -y upgrade && \
	apt-get -y install gcc-arm-embedded openocd tup python3.7 python3-yaml python3-jinja2 python3-jsonschema build-essential git time && \
	# Build step below does not know about debian's python naming schemme
	ln -s /usr/bin/python3.7 /usr/bin/python && \
	mkdir -p ODrive

WORKDIR ODrive/Firmware

# Must attach the firmware tree into the container
CMD \
	# Regenerate autogen/version.c
	mkdir -p autogen && \
	python ../tools/odrive/version.py \
	--output autogen/version.c && \
	# Regenerate python interface
	python interface_generator_stub.py \
	--definitions odrive-interface.yaml \
	--template ../tools/enums_template.j2 \
	--output ../tools/odrive/enums.py && \
	python interface_generator_stub.py \
	--definitions odrive-interface.yaml \
	--template ../tools/arduino_enums_template.j2 \
	--output ../Arduino/ODriveArduino/ODriveEnums.h && \
	# Hack around Tup's dependency on FUSE
	tup init && \
	tup generate build.sh && \
	./build.sh