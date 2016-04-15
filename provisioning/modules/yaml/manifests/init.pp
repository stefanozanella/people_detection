class yaml {
  package { ["libyaml-cpp0.5", "libyaml-cpp-dev"]:
    ensure => installed,
  }
}
