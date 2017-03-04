float low_pass(float last_output, float input, float CUTOFF, float SAMPLE_RATE)
{
    float RC = 1.0/(CUTOFF*2*3.14);
    float dt = 1.0/SAMPLE_RATE;
    float alpha = dt/(RC+dt);
    return last_output + alpha*(input - last_output);
}

/*float high_pass(float last_output, float input, float CUTOFF, float SAMPLE_RATE)
{
    float RC = 1.0/(CUTOFF*2*3.14);
    float dt = 1.0/SAMPLE_RATE;
    float alpha = RC/(RC + dt);
    return alpha * (last_output + input - last_input);
}*/
