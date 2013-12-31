#!/usr/bin/ruby

constants = {}
File.open(ARGV.first,'r') {|f|
  f.readlines.grep(/define KEY_/).each {|l| c,v = l.split()[1,2]; constants[c] = v}
}
constants.delete("KEY_F(n)")
constants.delete("KEY_CODE_YES")

constants.each {|k,v| constants[k] = Integer(v)}

F0 = constants["KEY_F0"]
1.upto(63) do |n|
  constants["KEY_F#{n}"] = F0 + n
end

puts constants.map{|k,v| k = k.gsub('KEY', 'KEYCODE'); "uint16 #{k}=#{v}"}.join("\n")
puts 
puts "uint16 code"
