# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure("2") do |config|
  config.vm.box = "ubuntu/trusty64"

  config.vm.provider "virtualbox" do |v|
    v.gui = true
    v.customize [ "modifyvm", :id, "--cpus", 2]
  end

  config.ssh.forward_x11 = true

  config.vm.provision :puppet do |puppet|
    puppet.manifests_path = "provisioning/manifests"
    puppet.module_path = "provisioning/modules"
    puppet.manifest_file  = "init.pp"
    puppet.options = "--debug --verbose"
  end
end
