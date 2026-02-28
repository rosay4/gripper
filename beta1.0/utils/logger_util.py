def setup_logger(level="warn"):
    import hblog

    config = hblog.example_config()
    config["root"]["level"] = level
    hblog.start(config)
    return hblog
