in ?= dataset/1.pcd

all: vagrant_box
	vagrant ssh -c "cd /vagrant && make local_build"

run: vagrant_box
	vagrant ssh -c "cd /vagrant && make local_run app=people_detector in=$(in)"

extract_faces: vagrant_box
	vagrant ssh -c "cd /vagrant && make local_run app=extract_faces in=$(in)"

sample: vagrant_box
	vagrant ssh -c "cd /vagrant && make local_rum app=sample_app in=$(in)"

clean: vagrant_box
	vagrant ssh -c "cd /vagrant && make local_clean"

ifneq ($(shell which vagrant),)
vagrant_box_is_up = $(strip $(shell vagrant status | grep running))
endif

vagrant_box:
ifeq ($(vagrant_box_is_up),)
	@vagrant up
endif

local_build: build bin
	@cd build && cmake .. && make 

local_run: local_build
	@bin/$(app) /vagrant/$(in)

build:
	mkdir build

bin:
	mkdir bin

local_clean:
	rm -rf bin build
