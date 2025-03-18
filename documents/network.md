
# NAT Network Setup with Netplan on Ubuntu 20.04

We have a flat network, with the Office and Robot lan connected. Will configure our Ubuntu Host computer network interfaces using netplan for IP forwarding with NAT, and disable (or remove) NetworkManager to avoid conflicts. We configure Lan clients with a static IP that uses the host’s NAT network as its gateway. We do this in several steps, so hopefully we do not get locked out.

> **Note:**  
> - **eno0:** Used for external connectivity (DHCP). Office lan. 
> - **eno1:** Internal NAT network interface (static IP: 10.42.0.1/16).  
> - **eno2:** Provision for RF routing (static IP: 192.168.10.1/16).  
> - **eno3:** Provision for SAT routing (static IP: 192.168.20.1/16). 
> - The Client Jetson will be configured with a static IP (10.42.2.2/16 and 10.42.2.1) and will use the gateway 10.42.0.1.
> - Rest of Lan's clients will be configured with static IP's in the 10.42.2.0/16 and will use the same gateway (10.42.0.1).

## Step 1. Disable or Remove the notorious NetworkManager

Since we are using `networkd` as our netplan renderer, disable NetworkManager to avoid conflicts.

**To disable NetworkManager:**

```bash
sudo systemctl stop NetworkManager
sudo systemctl disable NetworkManager
```

**Or remove it completely (if not needed):**

```bash
sudo apt remove network-manager
```

---

## Step 2. Add Static IP in `10.42.0.0` to existing on Host:

Create or edit the netplan configuration file (e.g., `/etc/netplan/01-network-manager-all.yaml`) with the following content:

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eno0: 
      dhcp4: true #Or current static 192.168.0.0 number and gateway
      addresses:
        - 10.42.0.1/16 #Router
        - 10.42.0.2/16 #add if also GCS of tail number 2. 
```
Apply the changes with:

```bash
sudo netplan apply
```
You might also need to go ```sudo systemctl restart systemd-networkd```.
Test the result with:
```bash
ip -4 a s # short for ip -4 address show
```
---

## Step 3: Change IP to a Number in `10.42.1.0` segment to all Lan Clients
Change all clients numbers to `10.42.1.x/16, GW:10.42.0.1`.
You should be still able to connect to them after the change. Test it.
If we are happy, we can leave it as is.

## Step 4. Configure the Client Jetson

Jetpack typically don't use netplan. You may use ```sudo nmtui``` to edit your connection. Alternatively identify your connection in ```/etc/NetworkManager/system-connections/``` and edit the ipv4 section:
```yaml
[ipv4]
address1=10.42.2.2/16,10.42.0.1 #tail number 2 ip, with host as gateway
address2=10.42.2.1/16           #provision for serving as gateway for the rest of the lan
dns=8.8.8.8;8.8.4.4;
method=manual                 #Dirty secret, if you set to auto, you'll have both DHCP and Static IP's. 
                              #We don't need it here but probably will not harm. 
```
Apply the changes on the client with: (is a leap of faith)

```bash
systemctl restart NetworkManager
```
---

## Step 5. Enable IP Forwarding and Set Up NAT on Host
### a. On the host, move the `10.42.0.0` numbers to a different interface:
Edit `netplan` file:
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eno0: 
      dhcp4: true #You do not need the static IP anymore
    eno1:
      - 10.42.0.1/16 #This host will serve as router
      - 10.42.0.2/16 #This host will serve as GCS of tail number 2.
      # - feel free to add here additional number such as defaults of Lan components
    eno2:
      - 192.168.10.1/16 #Provision for RF Net
    eno3: 
      - 192.168.20.1/16 #provision for Sat Net
```

### b. Enable IP Forwarding

Add the following line to `/etc/sysctl.conf`:

```
net.ipv4.ip_forward=1
```

Then reload with:

```bash
sudo sysctl -p
```

### c. Set Up NAT with iptables

Add the following iptables rule for masquerading:

```bash
sudo iptables -t nat -A POSTROUTING -o eno0 -j MASQUERADE
sudo iptables -A FORWARD -i eno1 -o eno0 -j ACCEPT
sudo iptables -A FORWARD -i eno0 -o eno1 -m state --state RELATED,ESTABLISHED -j ACCEPT

```

Save this rule permanently:

```bash
sudo apt install iptables-persistent
sudo netfilter-persistent save
```
*Now, move the Robot's Lan connection to `eno1`.*
---


## Step 5. Testing Connectivity

### From the Client Jetson
- **Ping the GCS IP:**
  ```bash
  ping -c 4 10.42.0.2
  ```
- **Ping the Gateway IP:**
  ```bash
  ping -c 4 10.42.0.1
  ```
- **Ping an External IP (e.g., Google DNS):**
  ```bash
  ping -c 4 8.8.8.8
  ```
- **Test DNS Resolution:**
  ```bash
  ping -c 4 google.com
  ```
- **Test `apt`:**
  ```bash
  sudo apt update
  ```
### From the Host:
- Ping the clients from the host.

---

## Summary

By following these steps, you have:

1. Disabled NetworkManager to prevent conflicts with `networkd` on Host.
2. Temporarily Added `10.42.0.0` segment IP to original interface on Host.
3. Configured Lan's clients with a static IP that uses the host’s internal NAT network as its gateway.
4. Moved `10.42.0.0` IP's to a new interface, and enabled IP forwarding and set up NAT (masquerading) on the host.
5. Moved Lan cable to new interface.
