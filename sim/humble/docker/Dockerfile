FROM nvidia/11.8.0-base-ubuntu22.04

# Handle user-prompt for Ubuntu installation (time zone selection & keyboard)
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone