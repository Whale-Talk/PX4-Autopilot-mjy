px4_add_module(
	MODULE drivers__ins__deta40
	MAIN deta40
	SRCS
		DETA40.cpp
		DETA40.hpp
		fdilink_protocol.h
	MODULE_CONFIG
		module.yaml
	)
