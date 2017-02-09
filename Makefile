in ?= dataset/1.pcd
out ?= dataset/positive

all: vagrant_box
	vagrant ssh -c "cd /vagrant && make local_build"

run: vagrant_box
	vagrant ssh -c "cd /vagrant && make local_run app=detection_test args='/vagrant/$(in)'"

train: vagrant_box
	vagrant ssh -c "cd /vagrant && make local_run app=train args='/vagrant/$(positive_set) /vagrant/$(negative_sample)'"

extract_samples: vagrant_box
	vagrant ssh -c "cd /vagrant && make local_run app=extract_samples args='/vagrant/$(in) /vagrant/$(out)'"

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
	@cd build && cmake .. && make $(app)

local_run: local_build
	@LC_ALL="C.UTF-8" time bin/$(app) $(args)

build:
	mkdir build

bin:
	mkdir bin

local_clean:
	rm -rf bin build
