import json
import nmap3
from blueoshelper import request
from typing import List, Optional
from loguru import logger

def check_for_proper_dvl(ip: str) -> bool:
  # Check the API for the product_name field
  url = f"http://{ip}/api/v1/about"
  try:
    return "DVL" in json.loads(request(url))["product_name"]
  except Exception as e:
    print(f"{ip} is not a dvl: {e}")
    return False
  json.loads(request(url))

def get_ips_wildcards(ips: List[str]):
    """
    Takes a list of ip strings and replaces the last field with * for it to be used by nmap
    """
    return ['.'.join([*(ip.split('.')[0:-1]),'*']) for ip in ips]

def find_the_dvl() -> Optional[str]:
    # The dvl always reports 192.168.194.95 on mdns, so we need to take drastic measures.
    # Nmap to the rescue!

    nmap = nmap3.Nmap()
    # generate the scan mask from our current ips
    networks = json.loads(request("http://127.0.0.1/cable-guy/v1.0/ethernet"))
    current_networks = [network["addresses"] for network in networks]
    # this looks like [{'ip': '192.168.2.2', 'mode': 'server'}]
    current_ips = []
    for network in current_networks:
      for entry in network:
        current_ips.append(entry["ip"])

    scans = get_ips_wildcards(current_ips)
    printf(f"Scanning: {scans} for DVLs")
    candidates = []
    for ip in scans:
      results = nmap.scan_top_ports(ip, args="-p 80 --open")
      for result in results:
        if result in current_ips:
          continue
        if result == "runtime" or result == "stats":
          continue
        candidates.append(result)

    print(f"candidates for being a dvl: {candidates}")

    for candidate in candidates:
      if check_for_proper_dvl(candidate):
        print(f"DVL found at {candidate}")
        return candidate
    return None