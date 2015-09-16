in ?= dataset/1.pcd

all: vagrant_box
	vagrant ssh -c "cd /vagrant && make local_build"

run: vagrant_box
	vagrant ssh -c "cd /vagrant && make local_run in=$(in)"

sample: vagrant_box
	vagrant ssh -c "cd /vagrant && make local_sample in=$(in)"

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
	@bin/people_detector /vagrant/$(in)

local_sample: local_build
	@bin/sample_app /vagrant/$(in)

build:
	mkdir build

bin:
	mkdir bin

local_clean:
	rm -rf bin build
