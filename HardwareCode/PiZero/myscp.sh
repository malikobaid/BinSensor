#!/usr/bin/expect -f

# connect via scp
spawn scp latest.jpg "malikobaid@178.159.10.106:/home/malikobaid/webcam/"
#######################
expect {
  -re ".*es.*o.*" {
    exp_send "yes\r"
    exp_continue
  }
  -re ".*sword.*" {
    exp_send "PASSWORD\r"
  }
}
interact
