Vagrant.configure(2) do |config|
  config.vm.box = 'ubuntu/trusty64'
  config.vm.box_check_update = false

  config.vm.synced_folder '.', '/home/vagrant/catkin_ws'

  config.vm.provider 'virtualbox' do |vb|
    vb.name = 'qtcopter_dev'
    vb.gui = true
    vb.memory = '1024'
  end
  config.vm.provision 'shell', path: 'bootstrap.sh', privileged: false

  # Add some files to use the VM without Vagrant.
  config.vm.provision 'file', source: 'ssh', destination: '/home/vagrant/Desktop/qtcopter_bootstrap'
  config.vm.provision 'file', source: 'update_qtcopter.sh', destination: '/home/vagrant/Desktop/update_qtcopter.sh'
end
