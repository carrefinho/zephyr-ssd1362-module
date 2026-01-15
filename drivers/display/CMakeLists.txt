# Copyright (c) 2024 Lukasz Hawrylko
# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_SSD1362 ssd1362.c)
