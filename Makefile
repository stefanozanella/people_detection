in ?= dataset/1.pcd

all: vagrant_box
	vagrant ssh -c "cd /vagrant && make local_build in=$(in)"

ifneq ($(shell which vagrant),)
vagrant_box_is_up = $(strip $(shell vagrant status | grep running))
endif

vagrant_box:
ifeq ($(vagrant_box_is_up),)
	@vagrant up
endif

local_build: build
	@cd build && cmake .. && make && cd ../bin && ./people_detector /vagrant/$(in)

build:
	mkdir build
