# Cessna 172

# Physical constants
rho0   = 1.2250          # air density at sea level [kg/m^3]
lambda_air = -0.0065         # temperature gradient in ISA [K/m]
Temp0  = 288.15          # temperature at sea level in ISA [K]
R      = 287.05          # specific gas constant [m^2/sec^2K]
g      = 9.80665            # [m/sec^2] (gravity constant)


# Cessna 172 parameters
S = 16.1651  # m^2
b = 10.9728  # m
c = 1.49352  # m

# Normalized moments of inertia
KY2 = 1.0  # Reference (actual Iyy = 1346.0 slug·ft²)
KX2 = 948.0/1346.0  
KZ2 = 1967.0/1346.0
KXZ = 0.0

# Longitudinal stability derivatives
CXu = -0.050   # Assumption
CXa = 0.200  # Assumption
CZu = -0.100  # Assumption
CZa = -4.50  # Assumption
CZadot = -1.7  # Negative of CLadot 
CZq = -3.9     # Negative of CLq
Cmu = 0.020   # Assumption
Cma = -1.8
Cmadot = -5.2
Cmq = -12.4

# Control derivatives - longitudinal
CX_de = 0.010   # This needs to be calculated or estimated
CZ_de = -0.347  # Negative of CL_de
CM_de = -1.28

# Lateral-directional stability derivatives
CYb = -0.31
CYp = -0.037
CYr = 0.21
CYbdot = 0.0  # Usually assumed to be zero for most aircraft
Clb = -0.089
Clp = -0.47
Clr = 0.08  # For low alpha
Cnb = 0.065
Cnp = -0.03
Cnr = -0.099
Cnbdot = 0.0  # Usually assumed to be zero

# Control derivatives - lateral-directional
CY_da = -0.05
CY_dr = 0.098
Cl_da = 0.23
Cl_dr = 0.0147
Cn_da = 0.0053
Cn_dr = -0.043
