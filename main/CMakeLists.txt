idf_component_register(SRCS "ws_echo_server.c"
                    "mcp4725.c"
                    INCLUDE_DIRS "."
                    EMBED_TXTFILES web/index.html)
