import math
from app.models import AccessPoint


def rssi_to_distance(rssi: float, ap: AccessPoint) -> float:
    """
    Log-distance path loss model.
        RSSI = TxPower - 10 * n * log10(d)
        d    = 10 ^ ((TxPower - RSSI) / (10 * n))
    """
    exponent = (ap.tx_power - rssi) / (10.0 * ap.path_loss_exponent)
    return math.pow(10.0, exponent)
