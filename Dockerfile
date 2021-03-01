FROM ubuntu:bionic as build
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -qq update
RUN apt-get install -qq -y git build-essential cmake
COPY . /usr/app/
WORKDIR /usr/app/build
RUN cmake .. && make

FROM ubuntu:bionic
COPY --from=build /usr/app/bin /usr/app
CMD /usr/app/TwoBarLinkage

