def monitor_following_distance(distances: list[float], speeds: list[float]) -> tuple[int, float, int]:
    """
    Analyzes following distance compared to safe distance (speed * 0.5).
    
    Args:
        distances (list[float]): Distance to the lead car at each second.
        speeds (list[float]): Speed of our car at each second.
        
    Returns:
        tuple[int, float, int]: (tailgating_seconds, minimum_distance, tailgate_incidents)
            - tailgating_seconds: total seconds distance was < safe distance
            - minimum_distance: absolute closest distance to the lead car (return 0.0 if empty list)
            - tailgate_incidents: number of separate instances the car started tailgating
    """
    if (len(distances)==0 or len(speeds)==0): 
        return (0,0,0);
    #car=0;
    tailgating_sec=0;
    min=float ('inf');
    safe=1;
    old_safe=1;
    incidents=0;
    
    for car in range (len(distances)):
        safe_dist= speeds[car]*0.5;

        old_safe=safe;
        if (distances[car]< safe_dist):
            tailgating_sec+=1;
            safe=0;
        else:
            safe=1;

        if (distances[car]<min):
            min=distances[car];

        if ((old_safe!=safe) and (safe==0)):
            incidents+=1;

    return (tailgating_secs, min, incidents);
    #pass
