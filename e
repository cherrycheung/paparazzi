commit 5c325f1c69deed553e2580a5103991328dfe9f62
Author: Felix Ruess <felix.ruess@gmail.com>
Date:   Thu Apr 2 14:02:58 2015 +0200

    [settings] add optional target attribute to settings

commit 70b08f099cef2360faae2daf1da7bf8fec3aa015
Author: Felix Ruess <felix.ruess@gmail.com>
Date:   Sun Aug 10 22:47:21 2014 +0200

    [rotorcraft] horizontal guidance reference refactor
    
    make the reference model adjustable at runtime via settings

commit 30d7a959c0855fccbcf8cf7cb393393e260c8c57
Author: Felix Ruess <felix.ruess@gmail.com>
Date:   Tue May 20 17:51:51 2014 +0200

    [rotorcraft] add some functions to set heading via flight plan

commit 5de51d35588fa0080db7b8416924a900b405b4e9
Author: Gautier Hattenberger <gautier.hattenberger@enac.fr>
Date:   Wed Apr 16 18:26:41 2014 +0200

    [guidance] fix IGAIN precision and add VGAIN
    
    based on #682
    
    this may introduce too large horizontal guidance IGAIN in rotorcraft
    airframe files

commit fff0528972df0a5e062dd8ac8d9f0334b45f0875
Author: Felix Ruess <felix.ruess@gmail.com>
Date:   Tue Mar 11 00:42:14 2014 +0100

    [rotorcraft] guidance_h_ref: dynacmically adjustable max_speed
    
    - set_max_speed via settings
    
    closes #664

commit 4fb484c3d3323af56795b035de2908247a3a141f
Author: Felix Ruess <felix.ruess@gmail.com>
Date:   Thu Feb 27 13:32:43 2014 +0100

    [ins] remove hf|vf_realign from global ins struct
    
    and fix some missing fuctions

commit 03234de546c2f94f5be2dcddf1966d160f8f8134
Author: Felix Ruess <felix.ruess@gmail.com>
Date:   Fri Sep 13 16:09:32 2013 +0200

    [rotorcraft][guidance_h] guidance_h_approx_force_by_thrust
    
    - enable/disable via settings
    - gets a better approximation of vertical thrust using guidance_v_thrust_coeff
    - vertical thrust is recomputed in guidance_h to take the latest thrust command after guidance_v was run

commit fcc3b270ed164cb37b1db9e3cb3d8c2a084cfd82
Author: Felix Ruess <felix.ruess@gmail.com>
Date:   Tue Aug 13 14:14:41 2013 +0200

    [rotorcraft] v_adapt: config improvements and refactor
    
    - GUIDANCE_V_ADAPT_INITIAL_HOVER_THROTTLE settable in % with default of 0.3
    - GUIDANCE_V_ADAPT_THROTTLE_ENABLED:
      - TRUE by default if GUIDANCE_V_NOMINAL_HOVER_THROTTLE was not defined
      - FALSE by default if only GUIDANCE_V_NOMINAL_HOVER_THROTTLE was set in airframe file
        (for backwards compatibily)
    - switching between nominal_trottle and the adaptive estimate during operation via settings

commit 370100b83ea4d29a1c06b1964a56520d500511d4
Author: Felix Ruess <felix.ruess@gmail.com>
Date:   Sat Apr 13 14:01:31 2013 +0200

    [rotorcraft] use horizontal guidance ref by default
    
    Also use the horizontal guidance reference for HOVER mode.
    Using the reference can be temporarly disabled via settings.
    The reference is then still computed to stay consistent, but not used.

commit e58c77f48c5434f8e94280a4feb69747d289c130
Author: Felix Ruess <felix.ruess@gmail.com>
Date:   Wed Sep 19 20:48:34 2012 +0200

    [subsystems] add vf_realign and hf_realign to ins struct

commit 8f4f57ba7f2e449257f52e158b35e87eae32d4ff
Author: Felix Ruess <felix.ruess@gmail.com>
Date:   Wed Jul 18 16:48:12 2012 +0200

    [rotorcraft guidance] make guidance_v_nominal_throttle a float value

commit 66a56778847fc1a0d29d8aed6857ec5f7c762391
Author: Gautier Hattenberger <gautier.hattenberger@enac.fr>
Date:   Wed Jul 18 15:37:36 2012 +0200

    [units] fixing unit definition for NOMINAL_HOVER_THROTTLE

commit 369ea304f237009bdd074bb0778bc49b9442e56f
Author: Felix Ruess <felix.ruess@gmail.com>
Date:   Tue May 22 19:56:23 2012 +0200

    [conf] removed guidance_h_psi_sp from settings, doesn't exist anymore

commit 09490b374c0b1e83a5d4cb19e8dd340578b21f83
Author: Gautier Hattenberger <gautier.hattenberger@enac.fr>
Date:   Tue May 8 00:21:29 2012 +0200

    [conf] update basic settings files for fw and rotorcraft
