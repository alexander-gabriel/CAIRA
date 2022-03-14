# Use the official image as a parent image.
FROM rasberry:latest

USER rasberry
WORKDIR /home/rasberry

EXPOSE 22 11311 11411 9090 62345
USER root
CMD ["/usr/sbin/sshd", "-D"]
