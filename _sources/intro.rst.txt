Introduction
======================

Controlling two motors through a f28379d microcontroller and two drv8305 motor drivers.

Installation
-------------
..  code-block:: bash
   :linenos:

    pip install git+https://github.com/cps-lab-saga/f28379d-drv8305-dual-comm.git@main

Basic usage
-------------

.. literalinclude :: ../../f28379d_drv8305_dual_comm/examples/basic_usage.py
   :language: python
   :linenos:


Read data using callback function
-----------------------------------

.. literalinclude :: ../../f28379d_drv8305_dual_comm/examples/read_data_cb.py
   :language: python
   :linenos:

Read data from queue
-----------------------------------

.. literalinclude :: ../../f28379d_drv8305_dual_comm/examples/read_data_queue.py
   :language: python
   :linenos: