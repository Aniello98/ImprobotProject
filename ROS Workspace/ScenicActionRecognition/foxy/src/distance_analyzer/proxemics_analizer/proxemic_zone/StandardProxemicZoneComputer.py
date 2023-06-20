from proxemics_analizer.proxemic_zone.ProxemicZoneComputer import ProxemicZoneComputer

import math

from impro_msgs.msg import ProxemicZone


class StandardProxemicZoneComputer(ProxemicZoneComputer):

    def compute_zone(self, actor_distance: float) -> int:
        if math.isnan(actor_distance) or actor_distance > 2.50:
            return 2
        elif 0 <= actor_distance <= 0.75:
            return 0
        else:
            return 1
