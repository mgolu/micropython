.. currentmodule:: network
.. _network.CELLNRF:

class CELLNRF -- control nRF91 Cellular device
==============================================

.. note::

    This class is an implementation of ``network.CELL`` specific to the 
    nRF91 family of devices. This is to be used when MicroPython is running
    on the nRF91's application processor. It is not for usage as a separate
    modem. That would require the SLM application running on the nRF91, and
    an AT command driver (not currently supported).

This class provides a driver for the Cellular modem in the nRF91. Example usage::

    import network
    import time
    # setup as a station
    nic = network.CELL()
    nic.active(True)
    nic.connect()
    while not nic.isconnected():
        time.sleep_ms(50)

    # now use socket as usual
    ...

Constructors
------------

.. class:: CELL()

   Create a CELL object.

Methods
-------

.. method:: CELL.connect()

   Start the connection process, per the configuration of the device. This method
   is non-blocking, so you need to use ``isconnected()`` to check if the connection 
   has been established.

.. method:: CELL.isconnected()

   Returns ``True`` if the modem is associated to the cellular network, ``False`` otherwise.

.. method:: CELL.status([param])

   Query dynamic status information of the interface. When called with no argument the return
   value describes the association status of the cellular interface. Values are:

   - ``0`` Not registered and not searching
   - ``1`` Registered to home network
   - ``2`` Not registered, searching for network
   - ``3`` Registration denied by network
   - ``4`` Unknown (for example, out of coverage)
   - ``5`` Registered to a roaming network
   - ``90`` Not registered due to a SIM card error

   *param* can be a string to retrieve another current status. Supported *params* are:

   - ``'mode'`` Retrieve the current network mode. Can be ``network.LTE_MODE_LTEM`` or 
      ``network.LTE_MODE_NBIOT``
   - ``'mccmnc'`` Retrieve the current Mobile Country Code and Mobile Network Code
   - ``'imei'`` Get the device's IMEI
   - ``'uuid'`` Get the device's UUID
   - ``'band'`` Retrieve the band for the current connection
   - ``'ipAddress'`` Retrieve the current IP Address of the device
   - ``'apn'`` Retrieve the current APN
   - ``'iccid'`` Retrieve the ICCID from the SIM card
   - ``'imsi'`` Retrieve the IMSI from the SIM card

.. method:: CELL.config('param')
            CELL.config(param=value, ...)

   Get or set general network interface parameters. 
       
   For setting parameters, the keyword argument syntax should be used, and 
   multiple parameters can be set at once. For example, 
       
   For querying, a parameter name should be quoted as a string, and only one
   parameter can be queried at a time::

   - ``mode``: A tuple with the allowed and preferred network capabilities.
      The first value in the tuple is the allowed values, and can be a combination
      of ``network.LTE_MODE_LTEM``, ``network.LTE_MODE_NBIOT``, and ``network.LTE_MODE_GPS``.

      The second value is the preferred connection, and it can be:
      - ``0``: for auto selection
      - ``network.LTE_MODE_LTEM``: to prefer LTE-M
      - ``network.LTE_MODE_NBIOT``: to prefer NBIOT
      - ``network.PLMN_PREF | network.LTE_MODE_LTEM``: to select the home network if possible,
         LTEM otherwise.
      - ``network.PLMN_PREF | network.LTE_MODE_NBIOT``: to select the home network if possible,
         NBIOT otherwise.

      
      For example, the following enables LTEM, NBIOT, and GPS, with auto preference:

       nic.config(mode=(network.LTE_MODE_LTEM | network.LTE_MODE_NBIOT | network.LTE_MODE_GPS, 0))

.. method:: CELL.irq(param=value)

   Set the notification interrupt handler and a mask of which interrupts should be sent. The
   following parameters are accepted:

   - ``handler``: The function to be called with the interrupt data
   - ``mask``: The interrupt mask
   - ``add_mask``: A mask to add interrupts to the ones currently configured
   - ``remove_mask``: A mask to remove interrupts from the ones currently configured

   For example:
      nic.irq(handler=function, mask=_IRQ_NW_REG_STATUS | _IRQ_RRC_UPDATE)
   
The irq codes are::

    from micropython import const
   _IRQ_NW_REG_STATUS       = const(0x1)
   _IRQ_PSM_UPDATE          = const(0x2)
   _IRQ_EDRX_UPDATE         = const(0x4)
   _IRQ_RRC_UPDATE          = const(0x8)
   _IRQ_CELL_UPDATE         = const(0x10)
   _IRQ_LTE_MODE_UPDATE     = const(0x20)
   _IRQ_TAU_PRE_WARN        = const(0x40)
   _IRQ_NEIGHBOR_CELL_MEAS  = const(0x80)


In order to save space in the firmware, these constants are not included on the
module. Add the ones that you need from the list above to your program.

.. method:: CELL.at('at_command')

   Send an AT command directly to the modem. The response from the modem will be returned
   as a string. Note that for commands that use a `%`, it needs to be sent as `%%`.


