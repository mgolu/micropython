.. currentmodule:: network
.. _network.WLAN:

class WLAN -- control built-in WiFi interfaces
==============================================

This class provides a driver for WiFi network processors.  Example usage::

    import network
    # enable station interface and connect to WiFi access point
    nic = network.WLAN(network.STA_IF)
    nic.active(True)
    nic.connect('your-ssid', 'your-key')
    # now use sockets as usual

Constructors
------------
.. class:: WLAN(interface_id)

Create a WLAN network interface object. Supported interfaces are
``network.STA_IF`` (station aka client, connects to upstream WiFi access
points) and ``network.AP_IF`` (access point, allows other WiFi clients to
connect). Availability of the methods below depends on interface type.
For example, only STA interface may `WLAN.connect()` to an access point.

Methods
-------

.. method:: WLAN.active([is_active])

    Activate ("up") or deactivate ("down") network interface, if boolean
    argument is passed. Otherwise, query current state if no argument is
    provided. Most other methods require active interface.

.. method:: WLAN.connect(ssid=None, key=None, *, bssid=None)

    Connect to the specified wireless network, using the specified key.
    If *bssid* is given then the connection will be restricted to the
    access-point with that MAC address (the *ssid* must also be specified
    in this case).

    For the Zephyr port, the *auth* parameter is required. You can use the
    integers from network.AUTH_**, or use the value returned in the scan, e.g::

     nic.connect('ssid', 'passphrase', network.AUTH_WPA2_PSK)

.. method:: WLAN.disconnect()

    Disconnect from the currently connected wireless network.

.. method:: WLAN.scan()

    Scan for the available wireless networks.
    Hidden networks -- where the SSID is not broadcast -- will also be scanned
    if the WLAN interface allows it.

    Scanning is only possible on STA interface. Returns list of tuples with
    the information about WiFi access points:

        (ssid, bssid, channel, RSSI, security, hidden)

    *bssid* is hardware address of an access point, in binary form, returned as
    bytes object. You can use `binascii.hexlify()` to convert it to ASCII form.

    There are five values for security:

        * 0 -- open
        * 1 -- WEP
        * 2 -- WPA-PSK
        * 3 -- WPA2-PSK
        * 4 -- WPA/WPA2-PSK
    
    For the Zephyr port, the values should be checked against the built-in values, and
    it can also be passed directly to the `connect()` function as the `auth` parameter:
        * network.AUTH_OPEN
        * network.AUTH_WPA2_PSK
        * network.AUTH_WPA2_PSK_SHA256
        * network.AUTH_WPA3_SAE

    and two for hidden:

        * 0 -- visible
        * 1 -- hidden

.. method:: WLAN.status([param])

    Return the current status of the wireless connection.

    When called with no argument the return value describes the network link status.
    The possible statuses are defined as constants:

        * ``STAT_IDLE`` -- no connection and no activity,
        * ``STAT_CONNECTING`` -- connecting in progress,
        * ``STAT_WRONG_PASSWORD`` -- failed due to incorrect password,
        * ``STAT_NO_AP_FOUND`` -- failed because no access point replied,
        * ``STAT_CONNECT_FAIL`` -- failed due to other problems,
        * ``STAT_GOT_IP`` -- connection successful.

    When called with one argument *param* should be a string naming the status
    parameter to retrieve.  Supported parameters in WiFI STA mode are: ``'rssi'``.

    In addition, the Zephyr port also supports the following parameters. All the parameters
    refer to the access point that the device is currently connected to.

        * ``'ssid'``: SSID of the current connection
        * ``'band'``: The frequency band being used, e.g. network.WIFI_BAND_2_4_GHZ
        * ``'link_mode'``: The Wi-Fi generation. For example, 6 for Wi-Fi 6 (802.11ax)
        * ``'channel'``: The channel being used
        * ``'mfp'``: The current Management Frame Protection option. 0 - disabled; 1 - optional; 2 - required
        * ``'security'``: The security being used.
        * ``'bssid'``: Hardware address of the access point, in binary form, returned as bytes object.
        * ``'dtim'``: The DTIM period of the access point.
        * ``'beacon_interval'``: The beacon interval of the access point, in ms.
        * ``'twt_capable'``: Whether the access point supports TWT for power savings.

.. method:: WLAN.isconnected()

    In case of STA mode, returns ``True`` if connected to a WiFi access
    point and has a valid IP address.  In AP mode returns ``True`` when a
    station is connected. Returns ``False`` otherwise.

.. method:: WLAN.ifconfig([(ip, subnet, gateway, dns)])

   Get/set IP-level network interface parameters: IP address, subnet mask,
   gateway and DNS server. When called with no arguments, this method returns
   a 4-tuple with the above information. To set the above values, pass a
   4-tuple with the required information.  For example::

    nic.ifconfig(('192.168.0.4', '255.255.255.0', '192.168.0.1', '8.8.8.8'))

.. method:: WLAN.config('param')
            WLAN.config(param=value, ...)

   Get or set general network interface parameters. These methods allow to work
   with additional parameters beyond standard IP configuration (as dealt with by
   `WLAN.ifconfig()`). These include network-specific and hardware-specific
   parameters. For setting parameters, keyword argument syntax should be used,
   multiple parameters can be set at once. For querying, parameters name should
   be quoted as a string, and only one parameter can be queries at time::

    # Set WiFi access point name (formally known as SSID) and WiFi channel
    ap.config(ssid='My AP', channel=11)
    # Query params one by one
    print(ap.config('ssid'))
    print(ap.config('channel'))

   Following are commonly supported parameters (availability of a specific parameter
   depends on network technology type, driver, and :term:`MicroPython port`).

   =============  ===========
   Parameter      Description
   =============  ===========
   mac            MAC address (bytes)
   ssid           WiFi access point name (string)
   channel        WiFi channel (integer)
   hidden         Whether SSID is hidden (boolean)
   security       Security protocol supported (enumeration, see module constants)
   key            Access key (string)
   hostname       The hostname that will be sent to DHCP (STA interfaces) and mDNS (if supported, both STA and AP). (Deprecated, use :func:`network.hostname` instead)
   reconnects     Number of reconnect attempts to make (integer, 0=none, -1=unlimited)
   txpower        Maximum transmit power in dBm (integer or float)
   pm             WiFi Power Management setting (see below for allowed values)
   wmm            0 for Legacy power save mode, 1 for Wireless Multimedia (WMM)
   wakeup         0 for wakeup on every DTIM beacon, 1 to set a listen interval and skip beacons
   listen_interval Number of beacons to skip. For example, setting this to 10 will sleep for approximately 1 minute.
   timeout_ms     The power save inactivity timer (in ms)          
   =============  ===========

.. note::
    ``wmm`` and ``listen_interval`` parameters can only be changed when the device is
    not connected to an access point.

Constants
---------

.. data:: WLAN.PM_PERFORMANCE
        WLAN.PM_POWERSAVE
        WLAN.PM_NONE

    Allowed values for the ``WLAN.config(pm=...)`` network interface parameter:

        * ``PM_PERFORMANCE``: enable WiFi power management to balance power
          savings and WiFi performance
        * ``PM_POWERSAVE``: enable WiFi power management with additional power
          savings and reduced WiFi performance
        * ``PM_NONE``: disable wifi power management
