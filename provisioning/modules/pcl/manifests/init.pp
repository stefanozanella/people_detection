class pcl {
  $deps = [
    'build-essential',
    'cmake',
    'clang',
    'libpcl-all',
  ]

  exec { 'apt-get update':
    command => '/usr/bin/apt-get update',
  }

  apt::ppa { 'ppa:v-launchpad-jochen-sprickerhof-de/pcl':
    before => Package[$deps],
  }

  file { 'bashrc':
    ensure  => present,
    path    => "/home/${user}/.bashrc",
    content => template('pcl/dot_bashrc.erb'),
  }

  package { $deps:
    ensure => installed,
  }

  Package {
    require => Exec['apt-get update'],
  }
}
