# Log file parser for diagnosing altitude limiters
#
# The input is a csv log file from a radiomaster tx16s running edge tx
#
# the basic idea is it looks on the throttle channel for the motor
# start. But because the altitude limiter is cutting the throttle we
# cant use that for motor cutoff.  So what we do look at the main
# battery voltage.  Once the motor cuts off the voltage jumps
# up. After the motor cuts we then look for the peak altitude we get
# from coasting up.
#
# hopefully this can help with tuning altitude limiter zoom calculation.

prev_volts = 100
prev_alt = 0
prev_vspd = 0
prev_throt = -2000
count = 1
mode = 1
max_alt = 0
estimate_overshoot = 0
base_alt = 0
cut_vspd = 0
start_count =0
cut_alt = 0

max_list = []

if ARGV.length == 1 then
  filename = ARGV[0]
else
  puts "Need to provide a filename to read"
end

File.foreach(filename, chomp: true) do |line|
   s = line.split(',');
  vspd = s[9].to_f
  volts = s[11].to_f
  throt = s[20].to_f
    alt = s[8].to_f - base_alt
  if ((prev_throt == -1024)  && (throt > -1024) && ( s[24].to_i == 1)) then
    puts "motor start"
    puts "altitude #{alt}"
    base_alt = alt
    start_count = count
  end
  alt = s[8].to_f - base_alt
  if ((mode == 1) && (alt > 40) && (volts - prev_volts > 0.4)) then
    puts "suspected motor cutoff"
    puts "prev #{prev_alt} #{prev_vspd} #{prev_volts}"
    puts "cur  #{alt} #{vspd} #{volts} #{count}"
    puts "run time #{count - start_count}"
    estimate_overshoot = prev_vspd
    cut_vspd = vspd
    cut_alt = alt
    max_alt = alt
    mode = 2
  end
  if (mode == 2)  then
    if (alt >= max_alt) then
      max_alt = alt
    else
      puts "max alt #{max_alt}"
      max_list.push(max_alt)
      puts "coasting #{max_alt - cut_alt} #{cut_vspd} #{estimate_overshoot}"
      mode = 1
      max_alt = 0
    end
  end
  count += 1
  prev_volts = volts
  prev_alt = alt
  prev_vspd = vspd
  prev_throt = throt
end
puts max_list.sort
puts " ---------------------"
puts max_list.sum / max_list.length
