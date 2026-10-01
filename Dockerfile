
FROM almalinux:10

# copy the source in a directory for testing
COPY ./ /app/test/
# create an empty directory for development where
# the host directory will have to be mounted
RUN mkdir /app/develop/

# install the necessary packages
RUN dnf install -y gcc-c++ \
				   clang-tools-extra \
				   make \
				   cmake \
				   gdb \
				   valgrind \
				   git \
				   python3-pip \
				   vim

WORKDIR /app
