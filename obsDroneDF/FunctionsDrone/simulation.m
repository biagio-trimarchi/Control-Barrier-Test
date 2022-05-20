function ds = simulation(t, s, params, sigma_handle, dsigma_handle, ddsigma_handle, dddsigma_handle)
    sigma = sigma_handle(t);
    dsigma = dsigma_handle(t);
    ddsigma = ddsigma_handle(t);
    dddsigma = dddsigma_handle(t);
    
    u = controllerDF(s, params, sigma, dsigma, ddsigma, dddsigma);
    ds = droneModel(s, params, u);
end